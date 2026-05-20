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


# >>> USER_CODE_START
BASE_LENGTH = 0.42
BASE_WIDTH = 0.18
BASE_PLATE_THICKNESS = 0.02
GUIDE_LENGTH = 0.34
GUIDE_WIDTH = 0.04
GUIDE_HEIGHT = 0.022
RAIL_LENGTH = 0.34
RAIL_WIDTH = 0.022
RAIL_HEIGHT = 0.025
RAIL_Y = 0.055

SLIDE_TRAVEL = 0.09

CARRIAGE_LENGTH = 0.10
CARRIAGE_WIDTH = 0.13
CARRIAGE_BODY_HEIGHT = 0.05
CARRIAGE_BODY_Z = 0.045
CARRIAGE_BRIDGE_LENGTH = 0.08
CARRIAGE_BRIDGE_WIDTH = 0.05
CARRIAGE_BRIDGE_HEIGHT = 0.008
CARRIAGE_FRONT_PLATE_THICKNESS = 0.016
CARRIAGE_FRONT_PLATE_X = 0.058
CARRIAGE_FRONT_AXIS_Z = 0.055

TOOL_HOUSING_LENGTH = 0.10
TOOL_HOUSING_WIDTH = 0.08
TOOL_HOUSING_HEIGHT = 0.07
TOOL_NOSE_LENGTH = 0.055
TOOL_NOSE_RADIUS = 0.024
HEAD_ROT_LIMIT = pi / 2.0


def _base_visual_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS / 2.0))
    )
    guide = (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS + GUIDE_HEIGHT / 2.0))
    )
    base = base.union(guide)

    for rail_y in (-RAIL_Y, RAIL_Y):
        rail = (
            cq.Workplane("XY")
            .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
            .translate((0.0, rail_y, BASE_PLATE_THICKNESS + RAIL_HEIGHT / 2.0))
        )
        base = base.union(rail)

    for support_x in (-0.16, 0.16):
        for support_y in (-RAIL_Y, RAIL_Y):
            pedestal = (
                cq.Workplane("XY").box(0.04, 0.03, 0.08).translate((support_x, support_y, 0.06))
            )
            base = base.union(pedestal)

    back_bulkhead = cq.Workplane("XY").box(0.03, 0.12, 0.05).translate((-0.18, 0.0, 0.045))
    return base.union(back_bulkhead)


def _carriage_visual_shape() -> cq.Workplane:
    bridge = (
        cq.Workplane("XY")
        .box(CARRIAGE_BRIDGE_LENGTH, CARRIAGE_BRIDGE_WIDTH, CARRIAGE_BRIDGE_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_BRIDGE_HEIGHT / 2.0))
    )
    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_BODY_Z))
    )
    front_plate = (
        cq.Workplane("XY")
        .box(CARRIAGE_FRONT_PLATE_THICKNESS, 0.085, 0.07)
        .translate((CARRIAGE_FRONT_PLATE_X, 0.0, 0.045))
    )
    top_cover = cq.Workplane("XY").box(0.05, 0.06, 0.018).translate((-0.01, 0.0, 0.074))
    carriage = bridge.union(body).union(front_plate).union(top_cover)

    for runner_y in (-RAIL_Y, RAIL_Y):
        runner = cq.Workplane("XY").box(0.09, 0.026, 0.022).translate((-0.004, runner_y, 0.019))
        carriage = carriage.union(runner)

    return carriage


def _tool_head_visual_shape() -> cq.Workplane:
    rear_flange = (
        cq.Workplane("XY").box(0.01, 0.08, TOOL_HOUSING_HEIGHT).translate((0.005, 0.0, 0.0))
    )
    housing = (
        cq.Workplane("XY")
        .box(0.09, TOOL_HOUSING_WIDTH, TOOL_HOUSING_HEIGHT)
        .translate((0.055, 0.0, 0.0))
    )
    top_cover = cq.Workplane("XY").box(0.05, 0.03, 0.018).translate((0.06, 0.0, 0.044))
    nose = cq.Workplane("YZ").circle(TOOL_NOSE_RADIUS).extrude(0.03).translate((0.10, 0.0, 0.0))
    chuck = cq.Workplane("YZ").circle(0.012).extrude(0.025).translate((0.13, 0.0, 0.0))
    return rear_flange.union(housing).union(top_cover).union(nose).union(chuck)


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_mounted_rotary_tool_head", assets=ASSETS)

    model.material("frame_gray", rgba=(0.48, 0.5, 0.54, 1.0))
    model.material("carriage_blue", rgba=(0.17, 0.34, 0.68, 1.0))
    model.material("tool_orange", rgba=(0.86, 0.45, 0.12, 1.0))

    base = model.part("base_frame")
    base.visual(_mesh(_base_visual_shape(), "base_frame.obj"), material="frame_gray")


    for rail_y in (-RAIL_Y, RAIL_Y):
        pass
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.08)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    carriage = model.part("slide_carriage")
    carriage.visual(_mesh(_carriage_visual_shape(), "slide_carriage.obj"), material="carriage_blue")



    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.06)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    tool = model.part("tool_head")
    tool.visual(_mesh(_tool_head_visual_shape(), "tool_head.obj"), material="tool_orange")



    tool.inertial = Inertial.from_geometry(
        Box((0.12, TOOL_HOUSING_WIDTH, TOOL_HOUSING_HEIGHT)),
        mass=0.95,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
    )

    model.articulation(
        "slide_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=25.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "head_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tool,
        origin=Origin(xyz=(0.066, 0.0, CARRIAGE_FRONT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-HEAD_ROT_LIMIT,
            upper=HEAD_ROT_LIMIT,
            effort=8.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=96, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "slide_axis",
        "slide_carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "head_spin",
        "tool_head",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_aabb_overlap("slide_carriage", "base_frame", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_gap("slide_carriage", "base_frame", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_origin_gap("tool_head", "base_frame", axis="z", min_gap=0.015)
    ctx.expect_aabb_overlap("tool_head", "base_frame", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("tool_head", "slide_carriage", axes="xy", max_dist=0.13)

    with ctx.pose(slide_axis=-SLIDE_TRAVEL):
        ctx.expect_aabb_overlap("slide_carriage", "base_frame", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_gap("slide_carriage", "base_frame", axis="z", max_gap=0.01, max_penetration=0.0)
        ctx.expect_origin_distance("slide_carriage", "base_frame", axes="xy", max_dist=0.10)
        ctx.expect_origin_gap("tool_head", "base_frame", axis="z", min_gap=0.015)

    with ctx.pose(slide_axis=SLIDE_TRAVEL):
        ctx.expect_aabb_overlap("slide_carriage", "base_frame", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_gap("slide_carriage", "base_frame", axis="z", max_gap=0.01, max_penetration=0.0)
        ctx.expect_origin_distance("slide_carriage", "base_frame", axes="xy", max_dist=0.10)
        ctx.expect_origin_gap("tool_head", "base_frame", axis="z", min_gap=0.015)

    with ctx.pose(head_spin=HEAD_ROT_LIMIT):
        ctx.expect_origin_gap("tool_head", "base_frame", axis="z", min_gap=0.015)
        ctx.expect_aabb_overlap("tool_head", "base_frame", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("tool_head", "slide_carriage", axes="xy", max_dist=0.13)

    with ctx.pose(head_spin=-HEAD_ROT_LIMIT):
        ctx.expect_origin_gap("tool_head", "base_frame", axis="z", min_gap=0.015)
        ctx.expect_aabb_overlap("tool_head", "base_frame", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("tool_head", "slide_carriage", axes="xy", max_dist=0.13)

    with ctx.pose(slide_axis=SLIDE_TRAVEL, head_spin=HEAD_ROT_LIMIT):
        ctx.expect_origin_gap("tool_head", "base_frame", axis="z", min_gap=0.015)
        ctx.expect_aabb_overlap("tool_head", "base_frame", axes="xy", min_overlap=0.03)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
