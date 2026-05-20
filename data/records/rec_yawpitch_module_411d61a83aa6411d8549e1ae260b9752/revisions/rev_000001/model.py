from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.28
BASE_DEPTH = 0.20
BASE_PLINTH_H = 0.022
BASE_HOUSING_H = 0.056
BODY_TOP_Z = BASE_PLINTH_H + BASE_HOUSING_H
FIXED_COLLAR_R = 0.044
FIXED_COLLAR_H = 0.014
YAW_FRAME_Z = BODY_TOP_Z + FIXED_COLLAR_H

YAW_DRUM_R = 0.052
YAW_DRUM_H = 0.018
YOKE_BLOCK_W = 0.086
YOKE_BLOCK_D = 0.050
YOKE_BLOCK_H = 0.018
CHEEK_THICKNESS = 0.012
CHEEK_INNER_GAP = 0.062
CHEEK_OUTER_HALF = CHEEK_INNER_GAP / 2.0 + CHEEK_THICKNESS
PITCH_AXIS_Z = 0.060

CRADLE_BODY_W = 0.042
CRADLE_BODY_D = 0.058
CRADLE_BODY_H = 0.040
CRADLE_CENTER_Y = 0.022
TRUNNION_R = 0.009
TRUNNION_FACE_R = 0.013
TRUNNION_FACE_T = CHEEK_INNER_GAP / 2.0 - CRADLE_BODY_W / 2.0

YAW_LIMIT = 3.0
PITCH_LOWER = -0.75
PITCH_UPPER = 1.00


def _base_body_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_PLINTH_H)
        .translate((0.0, 0.0, BASE_PLINTH_H / 2.0))
        .edges("|Z")
        .fillet(0.016)
    )
    upper = (
        cq.Workplane("XY")
        .box(0.224, 0.154, BASE_HOUSING_H)
        .translate((0.0, 0.0, BASE_PLINTH_H + BASE_HOUSING_H / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )
    collar = (
        cq.Workplane("XY")
        .circle(FIXED_COLLAR_R)
        .extrude(FIXED_COLLAR_H)
        .translate((0.0, 0.0, BODY_TOP_Z))
    )
    return lower.union(upper).union(collar)


def _yaw_stage_shape() -> cq.Workplane:
    drum = cq.Workplane("XY").circle(YAW_DRUM_R).extrude(YAW_DRUM_H)
    saddle = (
        cq.Workplane("XY")
        .box(YOKE_BLOCK_W, YOKE_BLOCK_D, YOKE_BLOCK_H)
        .translate((0.0, 0.0, YAW_DRUM_H + YOKE_BLOCK_H / 2.0))
    )
    cheek_left = (
        cq.Workplane("XY")
        .box(CHEEK_THICKNESS, 0.044, 0.054)
        .translate((CHEEK_OUTER_HALF - CHEEK_THICKNESS / 2.0, 0.0, 0.059))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(CHEEK_THICKNESS, 0.044, 0.054)
        .translate((-CHEEK_OUTER_HALF + CHEEK_THICKNESS / 2.0, 0.0, 0.059))
    )
    housing_left = (
        cq.Workplane("YZ")
        .center(0.0, PITCH_AXIS_Z)
        .circle(0.016)
        .extrude(CHEEK_THICKNESS)
        .translate((CHEEK_OUTER_HALF - CHEEK_THICKNESS, 0.0, 0.0))
    )
    housing_right = (
        cq.Workplane("YZ")
        .center(0.0, PITCH_AXIS_Z)
        .circle(0.016)
        .extrude(-CHEEK_THICKNESS)
        .translate((-CHEEK_OUTER_HALF + CHEEK_THICKNESS, 0.0, 0.0))
    )
    return drum.union(saddle).union(cheek_left).union(cheek_right).union(housing_left).union(housing_right)


def _pitch_cradle_shape() -> cq.Workplane:
    cradle = (
        cq.Workplane("XY")
        .box(CRADLE_BODY_W, CRADLE_BODY_D, CRADLE_BODY_H)
        .translate((0.0, CRADLE_CENTER_Y, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    top_pad = (
        cq.Workplane("XY")
        .box(0.040, 0.034, 0.006)
        .translate((0.0, CRADLE_CENTER_Y + 0.004, CRADLE_BODY_H / 2.0 + 0.003))
    )
    nose = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .rect(0.030, 0.024)
        .extrude(0.010)
        .translate((0.0, CRADLE_CENTER_Y + CRADLE_BODY_D / 2.0 + 0.005, 0.0))
    )
    left_trunnion = (
        cq.Workplane("YZ")
        .center(0.0, 0.0)
        .circle(TRUNNION_FACE_R)
        .extrude(TRUNNION_FACE_T)
        .translate((CRADLE_BODY_W / 2.0, 0.0, 0.0))
    )
    right_trunnion = (
        cq.Workplane("YZ")
        .center(0.0, 0.0)
        .circle(TRUNNION_FACE_R)
        .extrude(-TRUNNION_FACE_T)
        .translate((-CRADLE_BODY_W / 2.0, 0.0, 0.0))
    )
    return (
        cradle.union(top_pad)
        .union(nose)
        .union(left_trunnion)
        .union(right_trunnion)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_yaw_pitch_head")

    model.material("body_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("stage_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("cradle_alloy", rgba=(0.75, 0.78, 0.81, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_base_body_shape(), "body_shell"),
        material="body_paint",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, YAW_FRAME_Z)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, YAW_FRAME_Z / 2.0)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage_shell"),
        material="stage_gray",
        name="yaw_stage_shell",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.118, 0.070, 0.092)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_pitch_cradle_shape(), "pitch_cradle_shell"),
        material="cradle_alloy",
        name="cradle_shell",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.090, 0.066, 0.050)),
        mass=0.8,
        origin=Origin(xyz=(0.0, CRADLE_CENTER_Y, 0.0)),
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_FRAME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
            effort=24.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=14.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")

    ctx.check(
        "parts exist",
        all(part is not None for part in (body, yaw_stage, pitch_cradle)),
        details="One or more required parts are missing.",
    )
    ctx.check(
        "joint axes are service-head axes",
        yaw_joint.axis == (0.0, 0.0, 1.0) and pitch_joint.axis == (1.0, 0.0, 0.0),
        details=f"yaw_axis={yaw_joint.axis}, pitch_axis={pitch_joint.axis}",
    )
    ctx.expect_gap(
        yaw_stage,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="yaw stage seats on the body collar",
    )
    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        reason="The pitch cradle is represented with captured trunnion bearing faces nested into the compact yoke housings.",
    )
    ctx.expect_within(
        pitch_cradle,
        yaw_stage,
        axes="x",
        margin=0.0,
        name="pitch cradle stays captured between the yoke cheeks",
    )

    cradle_rest = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_shell")
    with ctx.pose({pitch_joint: 0.65}):
        cradle_pitched = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_shell")
    rest_center_z = None if cradle_rest is None else (cradle_rest[0][2] + cradle_rest[1][2]) / 2.0
    pitched_center_z = None if cradle_pitched is None else (cradle_pitched[0][2] + cradle_pitched[1][2]) / 2.0
    ctx.check(
        "positive pitch lifts the cradle nose",
        rest_center_z is not None
        and pitched_center_z is not None
        and pitched_center_z > rest_center_z + 0.006,
        details=f"rest_center_z={rest_center_z}, pitched_center_z={pitched_center_z}",
    )

    with ctx.pose({yaw_joint: pi / 2.0}):
        cradle_yawed = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_shell")
    yawed_center_x = None if cradle_yawed is None else (cradle_yawed[0][0] + cradle_yawed[1][0]) / 2.0
    ctx.check(
        "positive yaw swings the forward cradle toward negative x",
        yawed_center_x is not None and yawed_center_x < -0.010,
        details=f"yawed_center_x={yawed_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
