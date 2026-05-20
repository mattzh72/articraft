from __future__ import annotations

import inspect
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
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
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
import cadquery as cq


BASE_RADIUS = 0.11
BASE_THICKNESS = 0.018
PLINTH_RADIUS = 0.072
PLINTH_THICKNESS = 0.010
COLUMN_RADIUS = 0.018
COLUMN_HEIGHT = 0.110
SHOULDER_Z = BASE_THICKNESS + COLUMN_HEIGHT

LOWER_ARM_LENGTH = 0.280
UPPER_ARM_LENGTH = 0.235

SHOULDER_LIMITS = (-0.10, 1.15)
ELBOW_LIMITS = (-1.05, 1.05)


def _require_cadquery():
    return cq


def _mesh(name, shape):
    return mesh_from_cadquery(
        shape,
        MESH_DIR / name,
        tolerance=0.0008,
        angular_tolerance=0.08,
    )


def _build_base_shape():
    cq_mod = _require_cadquery()

    foot = cq_mod.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    plinth = (
        cq_mod.Workplane("XY")
        .circle(PLINTH_RADIUS)
        .extrude(PLINTH_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    column = (
        cq_mod.Workplane("XY")
        .circle(COLUMN_RADIUS)
        .extrude(COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    yoke_bridge = (
        cq_mod.Workplane("XY").box(0.032, 0.030, 0.014).translate((0.0, 0.0, SHOULDER_Z - 0.020))
    )
    yoke_left = cq_mod.Workplane("XY").box(0.024, 0.006, 0.038).translate((0.0, 0.015, SHOULDER_Z))
    yoke_right = (
        cq_mod.Workplane("XY").box(0.024, 0.006, 0.038).translate((0.0, -0.015, SHOULDER_Z))
    )
    return foot.union(plinth).union(column).union(yoke_bridge).union(yoke_left).union(yoke_right)


def _build_lower_arm_shape():
    cq_mod = _require_cadquery()

    shoulder_boss = (
        cq_mod.Workplane("XZ").circle(0.0185).extrude(0.022).translate((0.0, -0.011, 0.0))
    )
    main_beam = cq_mod.Workplane("XY").box(0.240, 0.016, 0.016).translate((0.130, 0.0, 0.0))
    elbow_bridge = cq_mod.Workplane("XY").box(0.050, 0.024, 0.012).translate((0.245, 0.0, 0.0))
    fork_left = cq_mod.Workplane("XY").box(0.018, 0.005, 0.034).translate((0.271, 0.013, 0.0))
    fork_right = cq_mod.Workplane("XY").box(0.018, 0.005, 0.034).translate((0.271, -0.013, 0.0))
    return shoulder_boss.union(main_beam).union(elbow_bridge).union(fork_left).union(fork_right)


def _build_upper_arm_frame_shape():
    cq_mod = _require_cadquery()

    elbow_boss = cq_mod.Workplane("XZ").circle(0.0175).extrude(0.020).translate((0.0, -0.010, 0.0))
    main_beam = cq_mod.Workplane("XY").box(0.182, 0.014, 0.014).translate((0.102, 0.0, 0.0))
    neck = cq_mod.Workplane("XY").box(0.050, 0.012, 0.010).translate((0.205, 0.0, -0.010))
    collar = (
        cq_mod.Workplane("YZ")
        .center(0.0, -0.020)
        .circle(0.016)
        .extrude(0.018)
        .translate((0.190, 0.0, 0.0))
    )
    return elbow_boss.union(main_beam).union(neck).union(collar)


def _build_shade_shape():
    cq_mod = _require_cadquery()

    rear_shell = (
        cq_mod.Workplane("YZ")
        .center(0.0, -0.025)
        .circle(0.022)
        .extrude(0.015, taper=10)
        .translate((0.194, 0.0, 0.0))
    )
    shade_body = (
        cq_mod.Workplane("YZ")
        .center(0.0, -0.025)
        .circle(0.043)
        .extrude(0.074, taper=-18)
        .translate((0.208, 0.0, 0.0))
    )
    front_rim = (
        cq_mod.Workplane("YZ")
        .center(0.0, -0.025)
        .circle(0.045)
        .extrude(0.005)
        .translate((0.282, 0.0, 0.0))
    )
    return rear_shell.union(shade_body).union(front_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reading_lamp", assets=ASSETS)

    model.material("powder_black", rgba=(0.12, 0.12, 0.14, 1.0))
    model.material("warm_brass", rgba=(0.66, 0.56, 0.29, 1.0))
    model.material("shade_cream", rgba=(0.93, 0.91, 0.84, 1.0))

    base = model.part("base")
    base.visual(_mesh("lamp_base.obj", _build_base_shape()), material="powder_black")


    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(_mesh("lower_arm.obj", _build_lower_arm_shape()), material="warm_brass")



    lower_arm.inertial = Inertial.from_geometry(
        Box((0.260, 0.024, 0.024)),
        mass=0.42,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    upper_assembly = model.part("upper_assembly")
    upper_assembly.visual(
        _mesh("upper_arm_frame.obj", _build_upper_arm_frame_shape()),
        material="warm_brass",
    )
    upper_assembly.visual(_mesh("lamp_shade.obj", _build_shade_shape()), material="shade_cream")



    upper_assembly.inertial = Inertial.from_geometry(
        Box((0.305, 0.072, 0.060)),
        mass=0.48,
        origin=Origin(xyz=(0.168, 0.0, -0.018)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lower_arm",
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=18.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_assembly",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_assembly",
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=12.0,
            velocity=1.6,
        ),
    )

    return model


def _pose(ctx: TestContext, positions):
    parameters = list(inspect.signature(ctx.pose).parameters.values())
    if any(param.kind == inspect.Parameter.VAR_KEYWORD for param in parameters):
        return ctx.pose(**positions)
    if len(parameters) == 1:
        return ctx.pose(positions)
    return ctx.pose(**positions)


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "base_to_lower_arm",
        "lower_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "lower_arm_to_upper_assembly",
        "upper_assembly",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    ctx.expect_origin_distance("lower_arm", "base", axes="xy", max_dist=0.18)
    ctx.expect_aabb_overlap("lower_arm", "base", axes="xy", min_overlap=0.015)
    ctx.expect_origin_distance("upper_assembly", "base", axes="xy", max_dist=0.48)

    with _pose(ctx, {"base_to_lower_arm": SHOULDER_LIMITS[1]}):
        ctx.expect_origin_gap("lower_arm", "base", axis="z", min_gap=0.02)
        ctx.expect_origin_gap("upper_assembly", "base", axis="z", min_gap=0.05)
        ctx.expect_aabb_gap("upper_assembly", "base", axis="z", max_gap=0.45, max_penetration=0.0)

    with _pose(
        ctx,
        {
            "base_to_lower_arm": 0.55,
            "lower_arm_to_upper_assembly": ELBOW_LIMITS[0],
        },
    ):
        ctx.expect_origin_gap("upper_assembly", "base", axis="z", min_gap=0.01)
        ctx.expect_origin_distance("upper_assembly", "lower_arm", axes="xy", max_dist=0.34)

    with _pose(
        ctx,
        {
            "base_to_lower_arm": SHOULDER_LIMITS[0],
            "lower_arm_to_upper_assembly": ELBOW_LIMITS[1],
        },
    ):
        ctx.expect_origin_distance("upper_assembly", "base", axes="xy", max_dist=0.55)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
