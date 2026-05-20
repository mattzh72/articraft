from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_PLATE_X = 0.180
SUPPORT_PLATE_Y = 0.120
SUPPORT_PLATE_T = 0.012
SUPPORT_BODY_X = 0.110
SUPPORT_BODY_Y = 0.075
SUPPORT_BODY_H = 0.036
SUPPORT_COLLAR_R = 0.036
SUPPORT_COLLAR_H = 0.010

YAW_FLANGE_R = 0.055
YAW_FLANGE_H = 0.010
YAW_NECK_R = 0.024
YAW_NECK_H = 0.014
YAW_BODY_X = 0.050
YAW_BODY_Y = 0.042
YAW_BODY_H = 0.032

PITCH_AXIS_Z = -0.060
TRUNNION_R = 0.010
TRUNNION_CLEARANCE = 0.0
ARM_T = 0.014
ARM_CENTER_X = 0.047
BOSS_R = 0.017
CRADLE_ARM_Y = 0.060
CRADLE_ARM_Z = 0.110
CRADLE_TRAY_X = 0.108
CRADLE_TRAY_Y = 0.070
CRADLE_TRAY_T = 0.009

YAW_LIMIT = pi * 0.90
PITCH_LOWER = -0.85
PITCH_UPPER = 0.65


def _make_top_support_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(SUPPORT_PLATE_X, SUPPORT_PLATE_Y, SUPPORT_PLATE_T)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, SUPPORT_COLLAR_H + SUPPORT_BODY_H + SUPPORT_PLATE_T / 2.0))
    )
    body = (
        cq.Workplane("XY")
        .box(SUPPORT_BODY_X, SUPPORT_BODY_Y, SUPPORT_BODY_H)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, SUPPORT_COLLAR_H + SUPPORT_BODY_H / 2.0))
    )
    collar = cq.Workplane("XY").circle(SUPPORT_COLLAR_R).extrude(SUPPORT_COLLAR_H)

    support = plate.union(body).union(collar)
    support = support.faces(">Z").workplane(centerOption="CenterOfMass").pushPoints(
        [(-0.060, -0.035), (-0.060, 0.035), (0.060, -0.035), (0.060, 0.035)]
    ).hole(0.009)
    return support


def _make_yaw_stage_shape() -> cq.Workplane:
    top_flange = (
        cq.Workplane("XY")
        .circle(YAW_FLANGE_R)
        .extrude(YAW_FLANGE_H)
        .faces(">Z")
        .edges()
        .fillet(0.002)
        .translate((0.0, 0.0, -YAW_FLANGE_H))
    )
    neck = (
        cq.Workplane("XY")
        .circle(YAW_NECK_R)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.022))
    )
    drop_link = (
        cq.Workplane("XY")
        .box(0.024, 0.018, 0.042)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, -0.043))
    )
    pivot_block = (
        cq.Workplane("XY")
        .box(0.036, 0.022, 0.016)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, PITCH_AXIS_Z))
    )
    cross_shaft = (
        cq.Workplane("YZ")
        .circle(TRUNNION_R)
        .extrude(2.0 * (ARM_CENTER_X + ARM_T / 2.0))
        .translate((-(ARM_CENTER_X + ARM_T / 2.0), 0.0, PITCH_AXIS_Z))
    )
    return top_flange.union(neck).union(drop_link).union(pivot_block).union(cross_shaft)


def _make_pitch_cradle_shape() -> cq.Workplane:
    left_arm = cq.Workplane("XY").box(ARM_T, CRADLE_ARM_Y, CRADLE_ARM_Z).translate((-ARM_CENTER_X, 0.0, -CRADLE_ARM_Z / 2.0))
    right_arm = cq.Workplane("XY").box(ARM_T, CRADLE_ARM_Y, CRADLE_ARM_Z).translate((ARM_CENTER_X, 0.0, -CRADLE_ARM_Z / 2.0))
    left_boss = cq.Workplane("YZ").circle(BOSS_R).extrude(ARM_T).translate((-(ARM_CENTER_X + ARM_T / 2.0), 0.0, 0.0))
    right_boss = cq.Workplane("YZ").circle(BOSS_R).extrude(ARM_T).translate((ARM_CENTER_X - ARM_T / 2.0, 0.0, 0.0))
    tray = cq.Workplane("XY").box(CRADLE_TRAY_X, CRADLE_TRAY_Y, CRADLE_TRAY_T).translate((0.0, 0.012, -0.106))
    front_lip = cq.Workplane("XY").box(CRADLE_TRAY_X, 0.010, 0.022).translate((0.0, 0.036, -0.095))

    cradle = left_arm.union(right_arm).union(left_boss).union(right_boss).union(tray).union(front_lip)
    left_hole = cq.Workplane("YZ").circle(TRUNNION_R + TRUNNION_CLEARANCE).extrude(ARM_T + 0.002).translate(
        (-(ARM_CENTER_X + ARM_T / 2.0) - 0.001, 0.0, 0.0)
    )
    right_hole = cq.Workplane("YZ").circle(TRUNNION_R + TRUNNION_CLEARANCE).extrude(ARM_T + 0.002).translate(
        (ARM_CENTER_X - ARM_T / 2.0 - 0.001, 0.0, 0.0)
    )
    tray_window = cq.Workplane("XY").box(0.066, 0.032, 0.011).translate((0.0, 0.010, -0.106))
    return cradle.cut(left_hole).cut(right_hole).cut(tray_window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_yaw_pitch_fixture")

    model.material("support_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("machined_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("fixture_black", rgba=(0.14, 0.15, 0.16, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(_make_top_support_shape(), "top_support"),
        material="support_gray",
        name="support_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage_shape(), "yaw_stage"),
        material="machined_silver",
        name="yaw_shell",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_make_pitch_cradle_shape(), "pitch_cradle"),
        material="fixture_black",
        name="cradle_shell",
    )

    model.articulation(
        "support_yaw",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-YAW_LIMIT, upper=YAW_LIMIT, effort=18.0, velocity=2.2),
    )
    model.articulation(
        "yaw_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=PITCH_LOWER, upper=PITCH_UPPER, effort=12.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("support_yaw")
    pitch_joint = object_model.get_articulation("yaw_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        reason="Pitch cradle rotates on captured trunnion pins seated in the yaw-stage pivot bearings.",
    )

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

    ctx.check(
        "yaw joint axis is vertical",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint axis is horizontal",
        tuple(round(v, 6) for v in pitch_joint.axis) == (1.0, 0.0, 0.0),
        f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "yaw joint has swing range",
        yaw_joint.motion_limits is not None
        and yaw_joint.motion_limits.lower is not None
        and yaw_joint.motion_limits.upper is not None
        and yaw_joint.motion_limits.lower < 0.0 < yaw_joint.motion_limits.upper,
        f"limits={yaw_joint.motion_limits}",
    )
    ctx.check(
        "pitch joint has bidirectional tilt range",
        pitch_joint.motion_limits is not None
        and pitch_joint.motion_limits.lower is not None
        and pitch_joint.motion_limits.upper is not None
        and pitch_joint.motion_limits.lower < 0.0 < pitch_joint.motion_limits.upper,
        f"limits={pitch_joint.motion_limits}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_contact(
            top_support,
            yaw_stage,
            contact_tol=0.0005,
            name="top support carries yaw stage",
        )
        ctx.expect_contact(
            yaw_stage,
            pitch_cradle,
            contact_tol=0.0008,
            name="yaw stage carries pitch cradle",
        )
        ctx.expect_gap(
            top_support,
            pitch_cradle,
            axis="z",
            min_gap=0.020,
            name="pitch cradle hangs below support",
        )
        ctx.expect_overlap(
            yaw_stage,
            pitch_cradle,
            axes="yz",
            min_overlap=0.020,
            name="pitch cradle envelopes the yaw carrier",
        )

    rest_aabb = ctx.part_world_aabb(pitch_cradle)
    ctx.check("pitch cradle rest pose bounds available", rest_aabb is not None, "missing rest-pose AABB")
    if rest_aabb is not None:
        rest_max_z = rest_aabb[1][2]
        with ctx.pose({yaw_joint: 0.85, pitch_joint: 0.55}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fixture clears in combined yaw-pitch pose")
            ctx.expect_contact(
                yaw_stage,
                pitch_cradle,
                contact_tol=0.0008,
                name="pitch cradle stays mounted while tilted",
            )
            tilted_aabb = ctx.part_world_aabb(pitch_cradle)
            ctx.check(
                "pitch motion raises the underslung cradle",
                tilted_aabb is not None and tilted_aabb[1][2] > rest_max_z + 0.010,
                f"rest_max_z={rest_max_z}, tilted_max_z={None if tilted_aabb is None else tilted_aabb[1][2]}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
