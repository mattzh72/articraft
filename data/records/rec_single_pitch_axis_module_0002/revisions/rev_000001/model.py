from __future__ import annotations

from math import pi

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


BASE_WIDTH = 0.44
BASE_DEPTH = 0.18
BASE_THICKNESS = 0.02

SUPPORT_SPAN = 0.34
SUPPORT_X = SUPPORT_SPAN / 2.0
SUPPORT_DEPTH = 0.08
SUPPORT_COLUMN_WIDTH = 0.05
SUPPORT_COLUMN_HEIGHT = 0.11
BEARING_BLOCK_WIDTH = 0.065
BEARING_BLOCK_DEPTH = 0.07
BEARING_BLOCK_HEIGHT = 0.04
LOWER_CROSS_WIDTH = 0.28
LOWER_CROSS_DEPTH = 0.05
LOWER_CROSS_HEIGHT = 0.036
UPPER_TIE_WIDTH = 0.32
UPPER_TIE_DEPTH = 0.016
UPPER_TIE_HEIGHT = 0.018
UPPER_TIE_Y = -0.028

SUPPORT_FRAME_PIVOT_Z = 0.14
GLOBAL_PIVOT_Z = BASE_THICKNESS + SUPPORT_FRAME_PIVOT_Z
SHAFT_RADIUS = 0.012
SHAFT_LENGTH = 0.34
CRADLE_TUBE_RADIUS = 0.021
CRADLE_TUBE_LENGTH = 0.27
PITCH_LOWER = -0.7
PITCH_UPPER = 0.8

PLATE_WIDTH = 0.26
PLATE_DEPTH = 0.18
PLATE_THICKNESS = 0.01
PLATE_TOP_Z_IN_CRADLE = -0.07


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((cx - length / 2.0, cy, cz))


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, *, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    rail_left = cq.Workplane("XY").box(0.33, 0.028, 0.014).translate((0.0, 0.055, 0.027))
    rail_right = cq.Workplane("XY").box(0.33, 0.028, 0.014).translate((0.0, -0.055, 0.027))
    return plate.union(rail_left).union(rail_right)


def _build_support_frame_shape() -> cq.Workplane:
    left_column = (
        cq.Workplane("XY")
        .box(
            SUPPORT_COLUMN_WIDTH,
            SUPPORT_DEPTH,
            SUPPORT_COLUMN_HEIGHT,
        )
        .translate((-SUPPORT_X, 0.0, SUPPORT_COLUMN_HEIGHT / 2.0))
    )
    right_column = (
        cq.Workplane("XY")
        .box(
            SUPPORT_COLUMN_WIDTH,
            SUPPORT_DEPTH,
            SUPPORT_COLUMN_HEIGHT,
        )
        .translate((SUPPORT_X, 0.0, SUPPORT_COLUMN_HEIGHT / 2.0))
    )
    left_block = (
        cq.Workplane("XY")
        .box(
            BEARING_BLOCK_WIDTH,
            BEARING_BLOCK_DEPTH,
            BEARING_BLOCK_HEIGHT,
        )
        .translate((-SUPPORT_X, 0.0, SUPPORT_FRAME_PIVOT_Z))
    )
    right_block = (
        cq.Workplane("XY")
        .box(
            BEARING_BLOCK_WIDTH,
            BEARING_BLOCK_DEPTH,
            BEARING_BLOCK_HEIGHT,
        )
        .translate((SUPPORT_X, 0.0, SUPPORT_FRAME_PIVOT_Z))
    )
    lower_cross = (
        cq.Workplane("XY")
        .box(
            LOWER_CROSS_WIDTH,
            LOWER_CROSS_DEPTH,
            LOWER_CROSS_HEIGHT,
        )
        .translate((0.0, 0.0, LOWER_CROSS_HEIGHT / 2.0))
    )
    upper_tie = (
        cq.Workplane("XY")
        .box(
            UPPER_TIE_WIDTH,
            UPPER_TIE_DEPTH,
            UPPER_TIE_HEIGHT,
        )
        .translate((0.0, UPPER_TIE_Y, SUPPORT_FRAME_PIVOT_Z))
    )
    shape = (
        left_column.union(right_column)
        .union(left_block)
        .union(right_block)
        .union(lower_cross)
        .union(upper_tie)
    )
    left_bore = _x_cylinder(0.018, 0.09, (-SUPPORT_X, 0.0, SUPPORT_FRAME_PIVOT_Z))
    right_bore = _x_cylinder(0.018, 0.09, (SUPPORT_X, 0.0, SUPPORT_FRAME_PIVOT_Z))
    return shape.cut(left_bore).cut(right_bore)


def _build_support_shaft_shape() -> cq.Workplane:
    shaft = _x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (0.0, 0.0, 0.0))
    left_collar = _x_cylinder(0.016, 0.01, (-0.162, 0.0, 0.0))
    right_collar = _x_cylinder(0.016, 0.01, (0.162, 0.0, 0.0))
    return shaft.union(left_collar).union(right_collar)


def _build_cradle_yoke_shape() -> cq.Workplane:
    tube = _x_cylinder(CRADLE_TUBE_RADIUS, CRADLE_TUBE_LENGTH, (0.0, 0.0, 0.0))
    left_hanger = cq.Workplane("XY").box(0.014, 0.10, 0.075).translate((-0.11, 0.0, -0.0375))
    right_hanger = cq.Workplane("XY").box(0.014, 0.10, 0.075).translate((0.11, 0.0, -0.0375))
    left_hub = _x_cylinder(0.028, 0.018, (-0.125, 0.0, 0.0))
    right_hub = _x_cylinder(0.028, 0.018, (0.125, 0.0, 0.0))
    center_mount = cq.Workplane("XY").box(0.16, 0.03, 0.008).translate((0.0, 0.0, -0.066))
    left_web = cq.Workplane("XY").box(0.012, 0.05, 0.062).translate((-0.07, 0.0, -0.037))
    right_web = cq.Workplane("XY").box(0.012, 0.05, 0.062).translate((0.07, 0.0, -0.037))
    return (
        tube.union(left_hanger)
        .union(right_hanger)
        .union(left_hub)
        .union(right_hub)
        .union(center_mount)
        .union(left_web)
        .union(right_web)
    )


def _build_equipment_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_WIDTH, PLATE_DEPTH, PLATE_THICKNESS)
        .translate((0.0, 0.0, -PLATE_THICKNESS / 2.0))
    )
    front_lip = cq.Workplane("XY").box(0.22, 0.012, 0.022).translate((0.0, 0.084, 0.011))
    rear_lip = cq.Workplane("XY").box(0.22, 0.012, 0.022).translate((0.0, -0.084, 0.011))
    shape = plate.union(front_lip).union(rear_lip)
    for x in (-0.07, 0.07):
        for y in (-0.05, 0.05):
            slot = cq.Workplane("XY").box(0.045, 0.012, 0.03).translate((x, y, -0.005))
            shape = shape.cut(slot)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_cradle", assets=ASSETS)
    model.material("frame_blue", rgba=(0.18, 0.33, 0.56, 1.0))
    model.material("dark_steel", rgba=(0.24, 0.24, 0.26, 1.0))
    model.material("plate_gray", rgba=(0.68, 0.70, 0.73, 1.0))

    base_plate = model.part("base_plate")
    _add_mesh_visual(base_plate, _build_base_plate_shape(), "base_plate.obj", material="frame_blue")

    base_plate.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    support_frame = model.part("support_frame")
    _add_mesh_visual(
        support_frame,
        _build_support_frame_shape(),
        "support_frame.obj",
        material="frame_blue",
    )




    support_frame.inertial = Inertial.from_geometry(
        Box((0.40, 0.09, 0.16)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    support_shaft = model.part("support_shaft")
    _add_mesh_visual(
        support_shaft,
        _build_support_shaft_shape(),
        "support_shaft.obj",
        material="dark_steel",
    )

    support_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    cradle_yoke = model.part("cradle_yoke")
    _add_mesh_visual(
        cradle_yoke,
        _build_cradle_yoke_shape(),
        "cradle_yoke.obj",
        material="dark_steel",
    )




    cradle_yoke.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.09)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    equipment_plate = model.part("equipment_plate")
    _add_mesh_visual(
        equipment_plate,
        _build_equipment_plate_shape(),
        "equipment_plate.obj",
        material="plate_gray",
    )

    equipment_plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_DEPTH, PLATE_THICKNESS)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -PLATE_THICKNESS / 2.0)),
    )

    model.articulation(
        "base_to_supports",
        ArticulationType.FIXED,
        parent=base_plate,
        child=support_frame,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "supports_to_shaft",
        ArticulationType.FIXED,
        parent=support_frame,
        child=support_shaft,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_FRAME_PIVOT_Z)),
    )
    model.articulation(
        "shaft_to_cradle",
        ArticulationType.REVOLUTE,
        parent=support_shaft,
        child=cradle_yoke,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=20.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "cradle_to_plate",
        ArticulationType.FIXED,
        parent=cradle_yoke,
        child=equipment_plate,
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z_IN_CRADLE)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "support_shaft",
        "cradle_yoke",
        reason="the rotating cradle tube is intentionally concentric with the stationary support shaft",
    )
    ctx.allow_overlap(
        "cradle_yoke",
        "support_frame",
        reason="the side bearing envelope is conservative and clips the yoke's swept AABB near the support cheeks",
    )
    ctx.allow_overlap(
        "equipment_plate",
        "support_frame",
        reason="the tilted suspended plate stays between the uprights, but its rotated AABB can conservatively intersect the support frame crossmembers",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_origin_gap("support_frame", "base_plate", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("equipment_plate", "base_plate", axis="z", min_gap=0.04)
    ctx.expect_origin_gap("support_shaft", "equipment_plate", axis="z", min_gap=0.05)
    ctx.expect_origin_distance("equipment_plate", "base_plate", axes="xy", max_dist=0.02)
    ctx.expect_origin_distance("support_shaft", "support_frame", axes="xy", max_dist=0.02)
    ctx.expect_origin_distance("cradle_yoke", "support_shaft", axes="xy", max_dist=0.04)
    ctx.expect_aabb_overlap("equipment_plate", "base_plate", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("equipment_plate", "base_plate", axis="z", max_gap=0.08, max_penetration=0.0)
    ctx.expect_aabb_gap("support_shaft", "equipment_plate", axis="z", max_gap=0.08, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "shaft_to_cradle",
        "cradle_yoke",
        world_axis="y",
        direction="positive",
        min_delta=0.04,
    )
    return ctx.report()


object_model = build_object_model()
