from __future__ import annotations

from math import pi

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
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_LENGTH = 0.38
BASE_WIDTH = 0.22
BASE_THICKNESS = 0.045

GUIDE_COLUMN_RADIUS = 0.02
GUIDE_COLUMN_HEIGHT = 0.62
GUIDE_COLUMN_OFFSET_X = 0.115

TOP_TIE_LENGTH = 0.31
TOP_TIE_WIDTH = 0.12
TOP_TIE_THICKNESS = 0.04

CARRIAGE_REST_Z = 0.18
CARRIAGE_TRAVEL = 0.30
CARRIAGE_COLUMN_SPAN = GUIDE_COLUMN_OFFSET_X * 2.0
BEARING_BLOCK_SPACING = 0.085


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage", assets=ASSETS)

    model.material("machine_base", rgba=(0.23, 0.24, 0.27, 1.0))
    model.material("machine_shadow", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("guide_steel", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("carriage_paint", rgba=(0.96, 0.48, 0.11, 1.0))
    model.material("bearing_housing", rgba=(0.33, 0.36, 0.39, 1.0))
    model.material("bushing_bronze", rgba=(0.69, 0.53, 0.25, 1.0))
    model.material("fastener_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_plinth")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="machine_base",
    )
    base.visual(
        Box((0.29, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, -0.065, 0.01)),
        material="machine_shadow",
    )
    base.visual(
        Box((0.29, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.065, 0.01)),
        material="machine_shadow",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    mast = model.part("service_mast")
    mast.visual(
        Box((0.07, 0.035, GUIDE_COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT / 2.0)),
        material="machine_base",
    )
    mast.visual(
        Box((0.16, 0.024, 0.10)),
        origin=Origin(xyz=(0.0, 0.018, 0.05)),
        material="machine_base",
    )
    mast.visual(
        Box((0.045, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, 0.008, 0.49)),
        material="machine_shadow",
    )
    mast.visual(
        Box((0.05, 0.018, 0.22)),
        origin=Origin(xyz=(-0.045, 0.008, 0.17)),
        material="machine_shadow",
    )
    mast.visual(
        Box((0.05, 0.018, 0.22)),
        origin=Origin(xyz=(0.045, 0.008, 0.17)),
        material="machine_shadow",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, GUIDE_COLUMN_HEIGHT)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.012, GUIDE_COLUMN_HEIGHT / 2.0)),
    )

    left_column = model.part("left_column")
    left_column.visual(
        Cylinder(radius=GUIDE_COLUMN_RADIUS, length=GUIDE_COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT / 2.0)),
        material="guide_steel",
    )
    left_column.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="bearing_housing",
    )
    left_column.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT - 0.008)),
        material="bearing_housing",
    )
    left_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=GUIDE_COLUMN_HEIGHT),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT / 2.0)),
    )

    right_column = model.part("right_column")
    right_column.visual(
        Cylinder(radius=GUIDE_COLUMN_RADIUS, length=GUIDE_COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT / 2.0)),
        material="guide_steel",
    )
    right_column.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="bearing_housing",
    )
    right_column.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT - 0.008)),
        material="bearing_housing",
    )
    right_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=GUIDE_COLUMN_HEIGHT),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT / 2.0)),
    )

    top_tie = model.part("top_tie")
    top_tie.visual(
        Box((TOP_TIE_LENGTH, TOP_TIE_WIDTH, TOP_TIE_THICKNESS)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.0, TOP_TIE_THICKNESS / 2.0)),
        material="machine_base",
    )
    top_tie.visual(
        Box((0.06, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="bearing_housing",
    )
    top_tie.visual(
        Box((0.06, 0.08, 0.05)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN, 0.0, 0.025)),
        material="bearing_housing",
    )
    top_tie.visual(
        Box((0.25, 0.012, 0.03)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.046, 0.028)),
        material="machine_shadow",
    )
    top_tie.inertial = Inertial.from_geometry(
        Box((TOP_TIE_LENGTH, TOP_TIE_WIDTH, 0.05)),
        mass=8.0,
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.0, 0.025)),
    )

    carriage = model.part("carriage")
    for x_pos, z_pos in (
        (0.0, 0.0),
        (0.0, BEARING_BLOCK_SPACING),
        (CARRIAGE_COLUMN_SPAN, 0.0),
        (CARRIAGE_COLUMN_SPAN, BEARING_BLOCK_SPACING),
    ):
        carriage.visual(
            Box((0.072, 0.062, 0.038)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material="bearing_housing",
        )
        carriage.visual(
            Cylinder(radius=0.026, length=0.03),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material="bushing_bronze",
        )

    carriage.visual(
        Box((0.018, 0.10, 0.125)),
        origin=Origin(xyz=(-0.027, 0.0, 0.0425)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.018, 0.10, 0.125)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN + 0.027, 0.0, 0.0425)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.272, 0.08, 0.055)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.0, 0.0425)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.22, 0.018, 0.10)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.049, 0.0425)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.018, 0.05, 0.10)),
        origin=Origin(xyz=(0.032, 0.020, 0.0425)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.018, 0.05, 0.10)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN - 0.032, 0.020, 0.0425)),
        material="carriage_paint",
    )
    carriage.visual(
        Box((0.10, 0.03, 0.02)),
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.070, 0.012)),
        material="machine_shadow",
    )
    for x_pos in (0.04, CARRIAGE_COLUMN_SPAN - 0.04):
        for z_pos in (0.015, 0.070):
            carriage.visual(
                Cylinder(radius=0.005, length=0.008),
                origin=Origin(xyz=(x_pos, 0.065, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
                material="fastener_black",
            )
    carriage.inertial = Inertial.from_geometry(
        Box((0.31, 0.11, 0.13)),
        mass=8.5,
        origin=Origin(xyz=(CARRIAGE_COLUMN_SPAN / 2.0, 0.012, 0.0425)),
    )

    model.articulation(
        "base_to_service_mast",
        ArticulationType.FIXED,
        parent="base_plinth",
        child="service_mast",
        origin=Origin(xyz=(0.0, -0.075, BASE_THICKNESS)),
    )
    model.articulation(
        "base_to_left_column",
        ArticulationType.FIXED,
        parent="base_plinth",
        child="left_column",
        origin=Origin(xyz=(-GUIDE_COLUMN_OFFSET_X, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "base_to_right_column",
        ArticulationType.FIXED,
        parent="base_plinth",
        child="right_column",
        origin=Origin(xyz=(GUIDE_COLUMN_OFFSET_X, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "left_column_to_top_tie",
        ArticulationType.FIXED,
        parent="left_column",
        child="top_tie",
        origin=Origin(xyz=(0.0, 0.0, GUIDE_COLUMN_HEIGHT)),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent="left_column",
        child="carriage",
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=800.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "carriage",
        "left_column",
        reason="left linear bearing sleeves intentionally envelop the left guide column",
    )
    ctx.allow_overlap(
        "carriage",
        "right_column",
        reason="right linear bearing sleeves intentionally envelop the right guide column",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_overlap("carriage", "base_plinth", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("carriage", "left_column", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("carriage", "right_column", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("carriage", "top_tie", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("top_tie", "left_column", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("top_tie", "right_column", axes="xy", min_overlap=0.03)
    ctx.expect_joint_motion_axis(
        "carriage_slide",
        "carriage",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    with ctx.pose(carriage_slide=0.0):
        ctx.expect_aabb_gap("carriage", "base_plinth", axis="z", max_gap=0.18, max_penetration=0.0)
        ctx.expect_aabb_gap("top_tie", "carriage", axis="z", max_gap=0.50, max_penetration=0.0)
    with ctx.pose(carriage_slide=CARRIAGE_TRAVEL):
        ctx.expect_aabb_overlap("carriage", "left_column", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("carriage", "right_column", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_gap("top_tie", "carriage", axis="z", max_gap=0.05, max_penetration=0.0)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
