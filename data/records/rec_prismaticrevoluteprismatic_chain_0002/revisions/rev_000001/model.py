from __future__ import annotations

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
LINEAR_JOINT_Z = 0.026
ELBOW_JOINT_Z = 0.052
TELESCOPE_JOINT_X = 0.11
TELESCOPE_JOINT_Z = 0.025


def _merge_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    merged = shapes[0]
    for shape in shapes[1:]:
        merged = merged.union(shape)
    return merged


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _base_main_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.72, 0.18, 0.018)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, -0.009))
    )
    center_strip = (
        cq.Workplane("XY")
        .box(0.54, 0.026, 0.014)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, 0.007))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(0.58, 0.018, 0.024)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.055, 0.012))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(0.58, 0.018, 0.024)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -0.055, 0.012))
    )
    return _merge_shapes(plate, center_strip, rail_left, rail_right)


def _base_cover_shape() -> cq.Workplane:
    motor_cover = (
        cq.Workplane("XY")
        .box(0.10, 0.14, 0.06)
        .edges("|Z")
        .fillet(0.01)
        .translate((-0.31, 0.0, 0.03))
    )
    idler_cover = (
        cq.Workplane("XY")
        .box(0.075, 0.10, 0.045)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.315, 0.0, 0.0225))
    )
    return _merge_shapes(motor_cover, idler_cover)


def _carriage_main_shape() -> cq.Workplane:
    saddle_plate = (
        cq.Workplane("XY")
        .box(0.17, 0.11, 0.018)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, 0.009))
    )
    pad_left = (
        cq.Workplane("XY")
        .box(0.12, 0.022, 0.016)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.042, 0.008))
    )
    pad_right = (
        cq.Workplane("XY")
        .box(0.12, 0.022, 0.016)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, -0.042, 0.008))
    )
    rotary_housing = cq.Workplane("XY").circle(0.038).extrude(0.026).translate((0.0, 0.0, 0.018))
    turntable_plate = (
        cq.Workplane("XY")
        .box(0.094, 0.094, 0.008)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, 0.048))
    )
    sensor_block = (
        cq.Workplane("XY")
        .box(0.028, 0.045, 0.022)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.055, 0.0, 0.029))
    )
    return _merge_shapes(
        saddle_plate, pad_left, pad_right, rotary_housing, turntable_plate, sensor_block
    )


def _carriage_cover_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.044, 0.082, 0.03)
        .edges("|Z")
        .fillet(0.004)
        .translate((-0.062, 0.0, 0.033))
    )


def _elbow_main_shape() -> cq.Workplane:
    rotary_base = cq.Workplane("XY").circle(0.05).extrude(0.03)
    shoulder = (
        cq.Workplane("XY")
        .box(0.10, 0.07, 0.04)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.06, 0.0, 0.02))
    )
    outer_sleeve = (
        cq.Workplane("XY")
        .box(0.24, 0.05, 0.05)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.19, 0.0, 0.025))
    )
    return _merge_shapes(rotary_base, shoulder, outer_sleeve)


def _elbow_cover_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.09, 0.055, 0.03)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.10, 0.0, 0.055))
    )


def _extension_main_shape() -> cq.Workplane:
    inner_tube = (
        cq.Workplane("XY")
        .box(0.24, 0.036, 0.036)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.12, 0.0, 0.0))
    )
    tube_cover = (
        cq.Workplane("XY")
        .box(0.09, 0.048, 0.02)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.15, 0.0, 0.028))
    )
    end_plate = (
        cq.Workplane("XY")
        .box(0.012, 0.08, 0.08)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.246, 0.0, 0.0))
    )
    return _merge_shapes(inner_tube, tube_cover, end_plate)


def _extension_tool_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.05, 0.032, 0.10)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.279, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_positioning_rotary_elbow_module", assets=ASSETS)

    model.material("steel", rgba=(0.66, 0.68, 0.72, 1.0))
    model.material("anodized", rgba=(0.36, 0.39, 0.43, 1.0))
    model.material("cover_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("tool_red", rgba=(0.78, 0.16, 0.14, 1.0))

    base = model.part("base_frame")
    _add_mesh_visual(base, _base_main_shape(), "base_frame_main.obj", "steel")
    _add_mesh_visual(base, _base_cover_shape(), "base_frame_covers.obj", "cover_black")






    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.18, 0.085)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    carriage = model.part("carriage")
    _add_mesh_visual(carriage, _carriage_main_shape(), "carriage_main.obj", "anodized")
    _add_mesh_visual(carriage, _carriage_cover_shape(), "carriage_cover.obj", "cover_black")





    carriage.inertial = Inertial.from_geometry(
        Box((0.17, 0.11, 0.06)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    elbow = model.part("elbow_stage")
    _add_mesh_visual(elbow, _elbow_main_shape(), "elbow_main.obj", "anodized")
    _add_mesh_visual(elbow, _elbow_cover_shape(), "elbow_cover.obj", "cover_black")



    elbow.inertial = Inertial.from_geometry(
        Box((0.32, 0.10, 0.08)),
        mass=1.6,
        origin=Origin(xyz=(0.14, 0.0, 0.035)),
    )

    extension = model.part("extension_inner")
    _add_mesh_visual(extension, _extension_main_shape(), "extension_main.obj", "steel")
    _add_mesh_visual(extension, _extension_tool_shape(), "extension_tool.obj", "tool_red")



    extension.inertial = Inertial.from_geometry(
        Box((0.30, 0.08, 0.10)),
        mass=0.9,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LINEAR_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.17, upper=0.18, effort=180.0, velocity=0.4),
    )
    model.articulation(
        "carriage_to_elbow",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow,
        origin=Origin(xyz=(0.0, 0.0, ELBOW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=40.0, velocity=2.0),
    )
    model.articulation(
        "elbow_to_extension",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=extension,
        origin=Origin(xyz=(TELESCOPE_JOINT_X, 0.0, TELESCOPE_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.16, effort=90.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "elbow_stage",
        "extension_inner",
        reason="the telescoping inner stage is intentionally nested inside the elbow sleeve",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.expect_origin_distance("carriage", "base_frame", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("carriage", "base_frame", axes="xy", min_overlap=0.10)
    ctx.expect_origin_gap("elbow_stage", "base_frame", axis="z", min_gap=0.015)
    ctx.expect_aabb_gap("elbow_stage", "carriage", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_aabb_overlap("elbow_stage", "carriage", axes="xy", min_overlap=0.04)
    ctx.expect_origin_distance("extension_inner", "elbow_stage", axes="xy", max_dist=0.20)
    ctx.expect_aabb_overlap("extension_inner", "elbow_stage", axes="xy", min_overlap=0.015)
    ctx.expect_joint_motion_axis(
        "base_to_carriage",
        "carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "carriage_to_elbow",
        "elbow_stage",
        world_axis="y",
        direction="positive",
        min_delta=0.20,
    )
    ctx.expect_joint_motion_axis(
        "elbow_to_extension",
        "extension_inner",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
