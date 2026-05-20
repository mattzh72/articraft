from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_L = 0.140
BASE_W = 0.110
BASE_T = 0.016
SHOULDER_Z = 0.135

SUPPORT_EAR_T = 0.012
SUPPORT_GAP = 0.022
SUPPORT_EAR_Y = (SUPPORT_GAP / 2.0) + (SUPPORT_EAR_T / 2.0)

UPPER_LENGTH = 0.235

ELBOW_GAP = 0.015
ELBOW_EAR_T = 0.011
ELBOW_EAR_Y = (ELBOW_GAP / 2.0) + (ELBOW_EAR_T / 2.0)

FORE_GUIDE_CENTER_X = 0.155
FORE_GUIDE_L = 0.090
FORE_GUIDE_W = 0.034
FORE_GUIDE_H = 0.030
STAGE_ORIGIN_X = 0.150

STAGE_BODY_L = 0.056
STAGE_BODY_W = 0.012
STAGE_BODY_H = 0.012
STAGE_TRAVEL = 0.045


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _support_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T)
        .translate((0.0, 0.0, BASE_T / 2.0))
    )

    tower = (
        cq.Workplane("XY")
        .box(0.048, 0.026, 0.104)
        .translate((-0.036, 0.0, BASE_T + 0.052))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.020, SUPPORT_EAR_T, 0.054)
        .translate((-0.010, SUPPORT_EAR_Y, SHOULDER_Z))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.020, SUPPORT_EAR_T, 0.054)
        .translate((-0.010, -SUPPORT_EAR_Y, SHOULDER_Z))
    )
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.058, BASE_T),
                (-0.028, BASE_T),
                (-0.004, 0.094),
                (-0.042, 0.094),
            ]
        )
        .close()
        .extrude(0.012, both=True)
    )

    return _union_all(base, tower, left_ear, right_ear, gusset)


def _upper_link_shape() -> cq.Workplane:
    shoulder_knuckle = (
        cq.Workplane("XY")
        .box(0.024, SUPPORT_GAP - 0.002, 0.030)
        .translate((0.012, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.186, 0.018, 0.028)
        .translate((0.117, 0.0, 0.0))
    )
    spine = (
        cq.Workplane("XY")
        .box(0.110, 0.028, 0.010)
        .translate((0.140, 0.0, -0.009))
    )
    elbow_block = (
        cq.Workplane("XY")
        .box(0.034, 0.020, 0.032)
        .translate((0.218, 0.0, 0.0))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.024, ELBOW_EAR_T, 0.042)
        .translate((0.223, ELBOW_EAR_Y, 0.0))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.024, ELBOW_EAR_T, 0.042)
        .translate((0.223, -ELBOW_EAR_Y, 0.0))
    )

    return _union_all(shoulder_knuckle, beam, spine, elbow_block, left_ear, right_ear)


def _forelink_shape() -> cq.Workplane:
    elbow_knuckle = (
        cq.Workplane("XY")
        .box(0.022, ELBOW_GAP - 0.002, 0.024)
        .translate((0.011, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.108, 0.016, 0.024)
        .translate((0.066, 0.0, 0.0))
    )
    transition = (
        cq.Workplane("XY")
        .box(0.030, 0.022, 0.026)
        .translate((0.116, 0.0, 0.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, FORE_GUIDE_W, FORE_GUIDE_H)
        .translate((0.117, 0.0, 0.0))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(FORE_GUIDE_L, 0.008, 0.020)
        .translate((FORE_GUIDE_CENTER_X, 0.011, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(FORE_GUIDE_L, 0.008, 0.020)
        .translate((FORE_GUIDE_CENTER_X, -0.011, 0.0))
    )
    return _union_all(elbow_knuckle, beam, transition, rear_bridge, left_rail, right_rail)


def _end_stage_shape() -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(STAGE_BODY_L, STAGE_BODY_W, STAGE_BODY_H)
        .translate((STAGE_BODY_L / 2.0, 0.0, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.020, 0.012, 0.012)
        .translate((STAGE_BODY_L + 0.010, 0.0, 0.0))
    )
    tool_pad = (
        cq.Workplane("XY")
        .box(0.020, 0.018, 0.018)
        .translate((STAGE_BODY_L + 0.026, 0.0, 0.0))
    )
    return _union_all(carriage, neck, tool_pad)


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        maxs[0] - mins[0],
        maxs[1] - mins[1],
        maxs[2] - mins[2],
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        0.5 * (mins[0] + maxs[0]),
        0.5 * (mins[1] + maxs[1]),
        0.5 * (mins[2] + maxs[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_arm_module")

    model.material("support_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("link_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("guide_dark", rgba=(0.19, 0.20, 0.23, 1.0))
    model.material("stage_light", rgba=(0.84, 0.86, 0.88, 1.0))

    support = model.part("support")
    support.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="support_gray",
        name="base_plate",
    )
    support.visual(
        Box((0.040, 0.030, 0.100)),
        origin=Origin(xyz=(-0.040, 0.0, BASE_T + 0.050)),
        material="support_gray",
        name="support_column",
    )
    support.visual(
        Box((0.030, SUPPORT_EAR_T, 0.052)),
        origin=Origin(xyz=(-0.015, SUPPORT_EAR_Y, SHOULDER_Z)),
        material="support_gray",
        name="left_shoulder_ear",
    )
    support.visual(
        Box((0.030, SUPPORT_EAR_T, 0.052)),
        origin=Origin(xyz=(-0.015, -SUPPORT_EAR_Y, SHOULDER_Z)),
        material="support_gray",
        name="right_shoulder_ear",
    )
    support.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.160)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Box((0.018, SUPPORT_GAP, 0.030)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material="link_silver",
        name="shoulder_knuckle",
    )
    upper_link.visual(
        Box((0.199, 0.018, 0.028)),
        origin=Origin(xyz=(0.1155, 0.0, 0.0)),
        material="link_silver",
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.020, ELBOW_GAP - 0.003, 0.032)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material="link_silver",
        name="elbow_block",
    )
    upper_link.visual(
        Box((0.020, ELBOW_EAR_T, 0.044)),
        origin=Origin(xyz=(0.225, ELBOW_EAR_Y, 0.0)),
        material="link_silver",
        name="left_elbow_ear",
    )
    upper_link.visual(
        Box((0.020, ELBOW_EAR_T, 0.044)),
        origin=Origin(xyz=(0.225, -ELBOW_EAR_Y, 0.0)),
        material="link_silver",
        name="right_elbow_ear",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.245, SUPPORT_GAP, 0.050)),
        mass=1.4,
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
    )

    forelink = model.part("forelink")
    forelink.visual(
        Box((0.016, ELBOW_GAP - 0.003, 0.024)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material="guide_dark",
        name="elbow_knuckle",
    )
    forelink.visual(
        Box((0.118, 0.016, 0.024)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material="guide_dark",
        name="fore_beam",
    )
    forelink.visual(
        Box((0.016, FORE_GUIDE_W, 0.026)),
        origin=Origin(xyz=(0.142, 0.0, 0.0)),
        material="guide_dark",
        name="guide_backstop",
    )
    forelink.visual(
        Box((0.084, 0.008, 0.020)),
        origin=Origin(xyz=(0.192, 0.010, 0.0)),
        material="guide_dark",
        name="left_guide_rail",
    )
    forelink.visual(
        Box((0.084, 0.008, 0.020)),
        origin=Origin(xyz=(0.192, -0.010, 0.0)),
        material="guide_dark",
        name="right_guide_rail",
    )
    forelink.visual(
        Box((0.040, 0.028, 0.006)),
        origin=Origin(xyz=(0.170, 0.0, 0.013)),
        material="guide_dark",
        name="top_guide_bridge",
    )
    forelink.inertial = Inertial.from_geometry(
        Box((0.205, FORE_GUIDE_W, FORE_GUIDE_H)),
        mass=1.0,
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
    )

    end_stage = model.part("end_stage")
    end_stage.visual(
        Box((STAGE_BODY_L, STAGE_BODY_W, STAGE_BODY_H)),
        origin=Origin(xyz=(STAGE_BODY_L / 2.0, 0.0, 0.0)),
        material="stage_light",
        name="stage_carriage",
    )
    end_stage.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(STAGE_BODY_L + 0.007, 0.0, 0.0)),
        material="stage_light",
        name="stage_neck",
    )
    end_stage.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(STAGE_BODY_L + 0.023, 0.0, 0.0)),
        material="stage_light",
        name="stage_tool",
    )
    end_stage.inertial = Inertial.from_geometry(
        Box((0.088, 0.012, 0.012)),
        mass=0.35,
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.65,
            upper=1.15,
            effort=60.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.20,
            upper=1.25,
            effort=35.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "stage_extension",
        ArticulationType.PRISMATIC,
        parent=forelink,
        child=end_stage,
        origin=Origin(xyz=(STAGE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_TRAVEL,
            effort=18.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    end_stage = object_model.get_part("end_stage")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    stage_extension = object_model.get_articulation("stage_extension")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(upper_link, support, name="support_carries_upper_link")
    ctx.expect_contact(forelink, upper_link, name="upper_link_carries_forelink")
    ctx.expect_contact(end_stage, forelink, name="forelink_guides_end_stage")

    ctx.expect_within(
        end_stage,
        forelink,
        axes="yz",
        margin=0.0,
        name="end_stage_stays_within_forelink_guide_width",
    )
    ctx.expect_overlap(
        end_stage,
        forelink,
        axes="yz",
        min_overlap=0.010,
        name="end_stage_has_visible_guide_engagement",
    )

    ctx.check(
        "shoulder_axis_matches_upward_pitch",
        shoulder_joint.axis == (0.0, -1.0, 0.0),
        details=f"unexpected shoulder axis: {shoulder_joint.axis}",
    )
    ctx.check(
        "elbow_axis_matches_upward_pitch",
        elbow_joint.axis == (0.0, -1.0, 0.0),
        details=f"unexpected elbow axis: {elbow_joint.axis}",
    )
    ctx.check(
        "terminal_joint_is_prismatic",
        stage_extension.articulation_type == ArticulationType.PRISMATIC,
        details=f"unexpected terminal articulation: {stage_extension.articulation_type}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, stage_extension: 0.0}):
        rest_stage_center = _aabb_center(ctx.part_world_aabb(end_stage))
        rest_upper_size = _aabb_size(ctx.part_world_aabb(upper_link))
        rest_fore_size = _aabb_size(ctx.part_world_aabb(forelink))
        rest_stage_size = _aabb_size(ctx.part_world_aabb(end_stage))

    with ctx.pose({shoulder_joint: 0.85, elbow_joint: 0.0, stage_extension: 0.0}):
        shoulder_lift_center = _aabb_center(ctx.part_world_aabb(end_stage))

    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.0, stage_extension: 0.0}):
        elbow_ref_center = _aabb_center(ctx.part_world_aabb(end_stage))
    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.85, stage_extension: 0.0}):
        elbow_lift_center = _aabb_center(ctx.part_world_aabb(end_stage))

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, stage_extension: STAGE_TRAVEL}):
        extended_stage_center = _aabb_center(ctx.part_world_aabb(end_stage))

    shoulder_ok = (
        rest_stage_center is not None
        and shoulder_lift_center is not None
        and shoulder_lift_center[2] > rest_stage_center[2] + 0.12
    )
    ctx.check(
        "positive_shoulder_raises_arm",
        shoulder_ok,
        details=(
            f"rest={rest_stage_center}, lifted={shoulder_lift_center}"
            if rest_stage_center is not None and shoulder_lift_center is not None
            else "missing stage AABB"
        ),
    )

    elbow_ok = (
        elbow_ref_center is not None
        and elbow_lift_center is not None
        and elbow_lift_center[2] > elbow_ref_center[2] + 0.06
    )
    ctx.check(
        "positive_elbow_raises_distal_stage",
        elbow_ok,
        details=(
            f"reference={elbow_ref_center}, lifted={elbow_lift_center}"
            if elbow_ref_center is not None and elbow_lift_center is not None
            else "missing stage AABB"
        ),
    )

    slide_ok = (
        rest_stage_center is not None
        and extended_stage_center is not None
        and extended_stage_center[0] > rest_stage_center[0] + (STAGE_TRAVEL * 0.95)
        and isclose(extended_stage_center[1], rest_stage_center[1], abs_tol=1e-6)
    )
    ctx.check(
        "prismatic_stage_extends_along_forelink",
        slide_ok,
        details=(
            f"retracted={rest_stage_center}, extended={extended_stage_center}"
            if rest_stage_center is not None and extended_stage_center is not None
            else "missing stage AABB"
        ),
    )

    smaller_stage_ok = (
        rest_upper_size is not None
        and rest_fore_size is not None
        and rest_stage_size is not None
        and rest_stage_size[0] < 0.7 * rest_upper_size[0]
        and rest_stage_size[0] < 0.7 * rest_fore_size[0]
        and rest_stage_size[1] <= rest_fore_size[1]
        and rest_stage_size[2] <= rest_fore_size[2]
    )
    ctx.check(
        "end_stage_is_visibly_smaller_than_main_links",
        smaller_stage_ok,
        details=(
            f"upper={rest_upper_size}, fore={rest_fore_size}, stage={rest_stage_size}"
            if rest_upper_size is not None
            and rest_fore_size is not None
            and rest_stage_size is not None
            else "missing link AABBs"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
