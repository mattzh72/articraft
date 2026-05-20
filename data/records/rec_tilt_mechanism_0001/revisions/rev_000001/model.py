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
BASE_LENGTH = 0.18
BASE_WIDTH = 0.12
BASE_THICKNESS = 0.012
PIVOT_Z = 0.115
TILT_LOWER = -0.55
TILT_UPPER = 0.90


def _make_stand_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS).edges("|Z").fillet(0.008)

    stem = cq.Workplane("XY").box(0.048, 0.030, 0.070).translate((-0.006, 0.0, 0.041))
    rear_bridge = cq.Workplane("XY").box(0.024, 0.064, 0.020).translate((-0.020, 0.0, 0.080))
    cheek_left = cq.Workplane("XY").box(0.034, 0.012, 0.040).translate((0.000, 0.032, 0.096))
    cheek_right = cq.Workplane("XY").box(0.034, 0.012, 0.040).translate((0.000, -0.032, 0.096))

    bushing_left = cq.Workplane("XY").box(0.012, 0.008, 0.012).translate((0.000, 0.018, PIVOT_Z))
    bushing_right = cq.Workplane("XY").box(0.012, 0.008, 0.012).translate((0.000, -0.018, PIVOT_Z))

    motor_pod = cq.Workplane("XZ").circle(0.017).extrude(0.014).translate((0.000, -0.061, PIVOT_Z))
    motor_cap = cq.Workplane("XZ").circle(0.010).extrude(0.004).translate((0.000, -0.067, PIVOT_Z))

    return (
        base.union(stem)
        .union(rear_bridge)
        .union(cheek_left)
        .union(cheek_right)
        .union(bushing_left)
        .union(bushing_right)
        .union(motor_pod)
        .union(motor_cap)
    )


def _make_cradle_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.140, 0.076, 0.006)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.000, 0.000, 0.014))
    )
    side_left = cq.Workplane("XY").box(0.126, 0.006, 0.018).translate((0.000, 0.035, 0.009))
    side_right = cq.Workplane("XY").box(0.126, 0.006, 0.018).translate((0.000, -0.035, 0.009))
    tilt_tube = cq.Workplane("XZ").circle(0.0075).extrude(0.094).translate((0.000, 0.047, 0.000))
    module_front = cq.Workplane("XY").box(0.034, 0.024, 0.014).translate((0.034, 0.000, 0.028))
    module_rear = cq.Workplane("XY").box(0.034, 0.024, 0.014).translate((-0.034, 0.000, 0.028))

    return (
        plate.union(side_left)
        .union(side_right)
        .union(tilt_tube)
        .union(module_front)
        .union(module_rear)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_tilt_cradle", assets=ASSETS)
    model.material("tower_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    model.material("plate_light", rgba=(0.74, 0.77, 0.80, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand_shape(), "stand.obj", assets=ASSETS),
        material="tower_dark",
    )






    stand.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.12)), mass=3.8, origin=Origin(xyz=(0.0, 0.0, 0.060))
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(_make_cradle_shape(), "cradle.obj", assets=ASSETS),
        material="plate_light",
    )

    cradle.inertial = Inertial.from_geometry(
        Box((0.140, 0.076, 0.040)), mass=1.1, origin=Origin(xyz=(0.0, 0.0, 0.020))
    )

    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=TILT_LOWER,
            upper=TILT_UPPER,
            effort=18.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.expect_origin_distance("cradle", "stand", axes="xy", max_dist=0.015)
    ctx.expect_origin_gap("cradle", "stand", axis="z", min_gap=0.08)
    ctx.expect_aabb_overlap("cradle", "stand", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_gap("cradle", "stand", axis="z", max_gap=0.020, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "tilt_joint",
        "cradle",
        world_axis="x",
        direction="positive",
        min_delta=0.015,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
