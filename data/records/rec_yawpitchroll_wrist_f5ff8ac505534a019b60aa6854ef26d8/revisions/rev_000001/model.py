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


HOUSING_BASE_X = 0.180
HOUSING_BASE_Y = 0.200
HOUSING_BASE_Z = 0.042
HOUSING_UPPER_Z = 0.056
HOUSING_TOP_Z = HOUSING_BASE_Z + HOUSING_UPPER_Z
YAW_ORIGIN_Z = 0.110

YAW_PLATTER_R = 0.062
YAW_PLATTER_H = 0.014
PITCH_ORIGIN_X = 0.050
PITCH_ORIGIN_Z = 0.040
PITCH_BOSS_R = 0.017
PITCH_BOSS_OUTER_Y = 0.048
PITCH_BOSS_INNER_Y = 0.015

ROLL_ORIGIN_X = 0.118
ROLL_REAR_FLANGE_R = 0.044


def _make_housing_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(HOUSING_BASE_X, HOUSING_BASE_Y, HOUSING_BASE_Z)
        .translate((0.0, 0.0, HOUSING_BASE_Z / 2.0))
    )

    upper = (
        cq.Workplane("XY")
        .rect(0.152, 0.132)
        .workplane(offset=HOUSING_UPPER_Z)
        .rect(0.112, 0.094)
        .loft(combine=False)
        .translate((-0.008, 0.0, HOUSING_BASE_Z))
    )

    front_saddle_cut = (
        cq.Workplane("XZ")
        .circle(0.058)
        .extrude(0.210)
        .translate((0.050, -0.105, 0.084))
    )

    bearing_ring = (
        cq.Workplane("XY")
        .circle(0.058)
        .extrude(YAW_ORIGIN_Z - HOUSING_TOP_Z)
        .translate((0.0, 0.0, HOUSING_TOP_Z))
    )

    return base.union(upper).cut(front_saddle_cut).union(bearing_ring)


def _make_yaw_stage_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(YAW_PLATTER_R).extrude(YAW_PLATTER_H)
    left_lower_support = (
        cq.Workplane("XY")
        .box(0.060, 0.026, 0.028)
        .translate((0.020, 0.036, 0.028))
    )
    right_lower_support = (
        cq.Workplane("XY")
        .box(0.060, 0.026, 0.028)
        .translate((0.020, -0.036, 0.028))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.032, 0.034, 0.056)
        .translate((PITCH_ORIGIN_X, 0.032, PITCH_ORIGIN_Z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.032, 0.034, 0.056)
        .translate((PITCH_ORIGIN_X, -0.032, PITCH_ORIGIN_Z))
    )

    return (
        platter
        .union(left_lower_support)
        .union(right_lower_support)
        .union(left_cheek)
        .union(right_cheek)
    )


def _make_pitch_support_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XZ").circle(0.016).extrude(0.015, both=True)
    neck = cq.Workplane("XY").box(0.028, 0.022, 0.024).translate((0.014, 0.0, 0.0))
    shell = (
        cq.Workplane("YZ")
        .rect(0.078, 0.094)
        .workplane(offset=ROLL_ORIGIN_X - 0.024)
        .rect(0.068, 0.082)
        .loft(combine=False)
        .translate((0.024, 0.0, 0.0))
    )
    front_mount = (
        cq.Workplane("YZ")
        .circle(0.054)
        .extrude(0.012)
        .translate((ROLL_ORIGIN_X - 0.012, 0.0, 0.0))
    )
    center_bore = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(ROLL_ORIGIN_X - 0.026)
        .translate((0.026, 0.0, 0.0))
    )

    return trunnion.union(neck).union(shell).union(front_mount).cut(center_bore)


def _make_roll_cartridge_shape() -> cq.Workplane:
    rear_flange = cq.Workplane("YZ").circle(ROLL_REAR_FLANGE_R).extrude(0.012)
    main_body = (
        cq.Workplane("YZ")
        .circle(0.037)
        .extrude(0.060)
        .translate((0.012, 0.0, 0.0))
    )
    taper = (
        cq.Workplane("YZ")
        .circle(0.037)
        .workplane(offset=0.032)
        .circle(0.028)
        .loft(combine=False)
        .translate((0.072, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(0.028)
        .extrude(0.020)
        .translate((0.104, 0.0, 0.0))
    )
    output_flange = (
        cq.Workplane("YZ")
        .circle(0.031)
        .extrude(0.010)
        .translate((0.124, 0.0, 0.0))
    )
    encoder_lump = (
        cq.Workplane("XY")
        .box(0.028, 0.022, 0.024)
        .translate((0.040, 0.012, 0.046))
    )

    return rear_flange.union(main_body).union(taper).union(nose).union(output_flange).union(encoder_lump)


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    *,
    mesh_name: str,
    material: str,
    visual_name: str,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_multi_axis_wrist")

    model.material("housing_graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("stage_cast", rgba=(0.47, 0.49, 0.52, 1.0))
    model.material("support_dark", rgba=(0.33, 0.35, 0.39, 1.0))
    model.material("cartridge_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    housing = model.part("housing")
    _add_mesh_visual(
        housing,
        _make_housing_shape(),
        mesh_name="housing_shell",
        material="housing_graphite",
        visual_name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_BASE_X, HOUSING_BASE_Y, YAW_ORIGIN_Z)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z / 2.0)),
    )

    yaw_stage = model.part("yaw_stage")
    _add_mesh_visual(
        yaw_stage,
        _make_yaw_stage_shape(),
        mesh_name="yaw_stage_shell",
        material="stage_cast",
        visual_name="yaw_stage_shell",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.124, 0.110, 0.072)),
        mass=2.2,
        origin=Origin(xyz=(0.030, 0.0, 0.036)),
    )

    pitch_support = model.part("pitch_support")
    _add_mesh_visual(
        pitch_support,
        _make_pitch_support_shape(),
        mesh_name="pitch_support_shell",
        material="support_dark",
        visual_name="pitch_support_shell",
    )
    pitch_support.inertial = Inertial.from_geometry(
        Box((ROLL_ORIGIN_X, 0.070, 0.064)),
        mass=1.5,
        origin=Origin(xyz=(ROLL_ORIGIN_X / 2.0, 0.0, 0.0)),
    )

    roll_cartridge = model.part("roll_cartridge")
    _add_mesh_visual(
        roll_cartridge,
        _make_roll_cartridge_shape(),
        mesh_name="roll_cartridge_shell",
        material="cartridge_silver",
        visual_name="roll_cartridge_shell",
    )
    roll_cartridge.inertial = Inertial.from_geometry(
        Box((0.134, 0.090, 0.090)),
        mass=1.0,
        origin=Origin(xyz=(0.067, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_yaw_stage",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=120.0, velocity=1.6),
    )
    model.articulation(
        "yaw_stage_to_pitch_support",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_support,
        origin=Origin(xyz=(PITCH_ORIGIN_X, 0.0, PITCH_ORIGIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.35, effort=70.0, velocity=1.8),
    )
    model.articulation(
        "pitch_support_to_roll_cartridge",
        ArticulationType.REVOLUTE,
        parent=pitch_support,
        child=roll_cartridge,
        origin=Origin(xyz=(ROLL_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-3.0, upper=3.0, effort=45.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_support = object_model.get_part("pitch_support")
    roll_cartridge = object_model.get_part("roll_cartridge")

    yaw_joint = object_model.get_articulation("housing_to_yaw_stage")
    pitch_joint = object_model.get_articulation("yaw_stage_to_pitch_support")
    roll_joint = object_model.get_articulation("pitch_support_to_roll_cartridge")

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
    ctx.allow_overlap(
        yaw_stage,
        pitch_support,
        elem_a="yaw_stage_shell",
        elem_b="pitch_support_shell",
        reason=(
            "The pitch-axis bearing fit is represented as an enclosed trunnion inside the "
            "yaw-stage carrier, so the simplified visual proxies intentionally interpenetrate."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "serial wrist joint axes match yaw-pitch-roll stack",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, -1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details=f"yaw={yaw_joint.axis}, pitch={pitch_joint.axis}, roll={roll_joint.axis}",
    )

    ctx.expect_contact(housing, yaw_stage, name="yaw stage seats on grounded housing")
    ctx.expect_contact(yaw_stage, pitch_support, name="pitch support seats on yaw trunnions")
    ctx.expect_contact(pitch_support, roll_cartridge, name="roll cartridge seats on pitch mount")

    with ctx.pose({pitch_joint: 0.75}):
        ctx.expect_contact(yaw_stage, pitch_support, name="pitch trunnions stay seated while pitching")
    with ctx.pose({roll_joint: 1.2}):
        ctx.expect_contact(
            pitch_support,
            roll_cartridge,
            name="roll cartridge stays seated while rolling",
        )

    def _span(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

    rest_output_pos = ctx.part_world_position(roll_cartridge)
    with ctx.pose({yaw_joint: 1.0}):
        yawed_output_pos = ctx.part_world_position(roll_cartridge)
    with ctx.pose({pitch_joint: 0.8}):
        pitched_output_pos = ctx.part_world_position(roll_cartridge)

    ctx.check(
        "yaw rotates output around the vertical axis",
        rest_output_pos is not None
        and yawed_output_pos is not None
        and yawed_output_pos[1] > rest_output_pos[1] + 0.10,
        details=f"rest={rest_output_pos}, yawed={yawed_output_pos}",
    )
    ctx.check(
        "positive pitch raises the cartridge",
        rest_output_pos is not None
        and pitched_output_pos is not None
        and pitched_output_pos[2] > rest_output_pos[2] + 0.05,
        details=f"rest={rest_output_pos}, pitched={pitched_output_pos}",
    )

    rest_pitch_aabb = ctx.part_world_aabb(pitch_support)
    rest_roll_aabb = ctx.part_world_aabb(roll_cartridge)
    with ctx.pose({roll_joint: pi / 2.0}):
        rolled_aabb = ctx.part_world_aabb(roll_cartridge)

    ctx.check(
        "roll cartridge is more compact than the pitch support",
        rest_pitch_aabb is not None
        and rest_roll_aabb is not None
        and _span(rest_roll_aabb, 1) < _span(rest_pitch_aabb, 1)
        and _span(rest_roll_aabb, 2) < _span(rest_pitch_aabb, 2),
        details=f"pitch_aabb={rest_pitch_aabb}, roll_aabb={rest_roll_aabb}",
    )
    ctx.check(
        "roll articulation changes cartridge orientation",
        rest_roll_aabb is not None
        and rolled_aabb is not None
        and rolled_aabb[1][2] < rest_roll_aabb[1][2] - 0.008,
        details=f"rest_aabb={rest_roll_aabb}, rolled_aabb={rolled_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
