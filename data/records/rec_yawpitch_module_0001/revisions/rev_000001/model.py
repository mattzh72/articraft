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
import cadquery as cq

from sdk import Cylinder


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_camera_gimbal_head", assets=ASSETS)

    model.material("anodized_black", rgba=(0.14, 0.14, 0.16, 1.0))
    model.material("machined_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("bearing_gray", rgba=(0.36, 0.38, 0.42, 1.0))

    base_disk = cq.Workplane("XY").circle(0.040).extrude(0.010)
    base_pedestal = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.019))
        .box(0.050, 0.038, 0.018)
        .edges("|Z")
        .fillet(0.003)
    )
    base_collar = cq.Workplane("XY", origin=(0.0, 0.0, 0.028)).circle(0.018).extrude(0.012)
    base_shape = base_disk.union(base_pedestal).union(base_collar)

    yaw_turntable = cq.Workplane("XY").circle(0.028).extrude(0.006)
    yaw_body = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.019))
        .box(0.052, 0.030, 0.022)
        .edges("|Z")
        .fillet(0.003)
    )
    yaw_support_pos = (
        cq.Workplane("XY", origin=(0.0, 0.024, 0.036))
        .box(0.014, 0.010, 0.030)
        .edges("|Z")
        .fillet(0.002)
    )
    yaw_support_neg = (
        cq.Workplane("XY", origin=(0.0, -0.024, 0.036))
        .box(0.014, 0.010, 0.030)
        .edges("|Z")
        .fillet(0.002)
    )
    yaw_shape = yaw_turntable.union(yaw_body).union(yaw_support_pos).union(yaw_support_neg)

    pitch_axle = cq.Workplane(
        obj=cq.Solid.makeCylinder(
            0.006,
            0.036,
            cq.Vector(0.0, -0.018, 0.0),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )
    pitch_hub = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.0))
        .box(0.024, 0.024, 0.016)
        .edges("|Z")
        .fillet(0.002)
    )
    pitch_web = (
        cq.Workplane("XY", origin=(-0.008, 0.0, 0.015))
        .box(0.012, 0.022, 0.030)
        .edges("|Z")
        .fillet(0.002)
    )
    pitch_plate = (
        cq.Workplane("XY", origin=(0.016, 0.0, 0.031))
        .box(0.055, 0.040, 0.006)
        .edges("|Z")
        .fillet(0.0015)
    )
    pitch_nose = (
        cq.Workplane("XY", origin=(0.034, 0.0, 0.020))
        .box(0.016, 0.024, 0.014)
        .edges("|Z")
        .fillet(0.0015)
    )
    pitch_shape = pitch_axle.union(pitch_hub).union(pitch_web).union(pitch_plate).union(pitch_nose)

    base = model.part("base")
    base.visual(_mesh(base_shape, "gimbal_base.obj"), material="anodized_black")



    base.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 0.040)),
        mass=1.10,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(_mesh(yaw_shape, "gimbal_yaw_stage.obj"), material="bearing_gray")




    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.052)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(_mesh(pitch_shape, "gimbal_pitch_frame.obj"), material="machined_aluminum")




    pitch_frame.inertial = Inertial.from_geometry(
        Box((0.065, 0.040, 0.040)),
        mass=0.46,
        origin=Origin(xyz=(0.012, 0.0, 0.018)),
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=6.0, velocity=2.5),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.35, effort=4.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("yaw_stage", "base", axes="xy", max_dist=0.003)
    ctx.expect_aabb_overlap("yaw_stage", "base", axes="xy", min_overlap=0.025)
    ctx.expect_aabb_gap("yaw_stage", "base", axis="z", max_gap=0.006, max_penetration=0.0)

    ctx.expect_origin_gap("pitch_frame", "base", axis="z", min_gap=0.02)
    ctx.expect_aabb_overlap("pitch_frame", "yaw_stage", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_gap("pitch_frame", "base", axis="z", max_gap=0.080, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "yaw_joint",
        "pitch_frame",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "pitch_joint",
        "pitch_frame",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
